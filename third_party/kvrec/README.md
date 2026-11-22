# kvrec

自描述二进制键值记录容器（轻量 JSON 替代）——从 [neuralnet.cpp](https://github.com/neuralnet-cpp-dev/neuralnet.cpp) 的 KVRecord 独立而来。header-only，命名空间 `kvrec`。

## 内容

| 头 | 提供 |
|----|------|
| `<kvrec/kvrecord.hpp>` | `kvrec::KeyValueRecord`（UInt / Str / UIntArray / 嵌套 Record）、`KvReadAt`、`kvrec_memory_source` |

**v1**（`serialize()`）：内联布局，无版本哨兵——模型 spec 头等既有格式继续用；`Record` 字段 v1 不支持（跳过不写）。

**v2**（`serialize_v2(base_offset=0)`）：自包含字节块 = 字段表 + 连续数据块；字段地址 = `base_offset` + 块内相对位移；嵌套 Record 子块递归序列化，addr 恒为**文件绝对偏移**（大文件可经 `parse_at(ReadAt, offset)` 流式解析）。

**版本自举**：`parse` 看哨兵字段 `kvrec_version`——无 = v1，有 = 按版本分派。未知字段类型按长度跳过（前向兼容）。⚠ `kvrec_version` 是 v2 保留键名，业务字段不得使用。

## 用法

```cmake
add_subdirectory(path/to/kvrec)          # 或 FetchContent；需先有 exstl::exstl
target_link_libraries(app PRIVATE kvrec::kvrec)
```

```cpp
#include <kvrec/kvrecord.hpp>

kvrec::KeyValueRecord rec;
rec.set("answer", std::uint64_t{42});
rec.set("name", std::string{"mnist"});
rec.set("shape", std::vector<std::uint64_t>{1, 28, 28});

std::uint64_t v = 0;
bool ok = rec.get("answer", v);          // ok && v == 42

std::string bytes = rec.serialize();     // v1
auto parsed = kvrec::KeyValueRecord::parse(bytes);  // exstl::Result<KeyValueRecord>
```

## 构建与测试（standalone）

```bash
# 需 sibling exstl（../exstl），或 find_package(exstl)
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

被主仓 `add_subdirectory` 时默认不建测试（`KVREC_ENABLE_TESTS=OFF`）。

## 依赖

- [exSTL](https://github.com/neuralnet-cpp-dev/exstl)（`exstl::Result` / `EX_*` 宏）
- C++23
- 零第三方库

格式规范与 Python 参考实现见 neuralnet.cpp 仓 `docs/development/10-kvrecord-format-spec.md` 与 `scripts/kvrecord.py`。

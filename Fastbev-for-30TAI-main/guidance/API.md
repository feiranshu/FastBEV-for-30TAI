# 运行时API相关说明
## 说明
本文档只针对部分C程序API接口进行说明，详细的API接口可以参考三个目录，第一个目录是 根目录/guidance/API_instruction  第二个是 根目录/deploy/deps/modelzoo_utils/include 第三个是根目录/docs/API_REFERENCE.md(自定义接口)
## 设备管理相关接口
仿照我写的pipeline.cpp即可，也可以参考实际API

## 神经网络相关接口
可以仿照我写的pipeline.cpp即可，也可以参考实际API
### Function getJrPath

Defined in [icraft_utils.hpp](./icraft_utils.hpp)

`std::string getJrPath(const std::string& run_backend, std::string& folderPath, std::string targetFileName)`

- 参数:

  **run_backend** – 是否是仿真(host)或运行至指定后端(buyi/zg330)

  **folderPath** – 指定模型文件所在的文件夹

  **targetFileName** – 指定模型的阶段

- 返回:

  指定文件夹中对应阶段的json文件路径

### Function loadNetwork

Defined in [icraft_utils.hpp](./icraft_utils.hpp)

`Network loadNetwork(const std::string& JSON_PATH, const std::string& RAW_PATH)`

- 参数:

  **JSON_PATH** – Json文件路径

  **RAW_PATH** – 指定raw的文件路径

### Function getOutputNormratio

Defined in [icraft_utils.hpp](./icraft_utils.hpp)

`std::vector<float> getOutputNormratio(icraft::xir::NetworkView network)`

- 参数:

  **network** – 网络对象（可以是network也可以是networkview）

- 返回:

  对应传入网络的输出数据的Normratio

- 说明:

注意输入网络要为实际需要的结构，保证拿到正确位置的Normratio

### Function getInputNormratio

Defined in [icraft_utils.hpp](./icraft_utils.hpp)

`std::vector<float> getInputNormratio(icraft::xir::NetworkView network)`

- 参数:

  **network** – 网络对象（可以是network也可以是networkview）

- 返回:

  对应传入网络的输入数据的Normratio

- 说明:

注意输入网络要为实际需要的结构，保证拿到正确位置的Normratio

### Class Netinfo

用network去初始化该类，可以获得对应网络的输入输出维度信息，fpga算子使用个情况，网络输出的量化scale信息等。


## PLDDR以及寄存器的内存分配与读写API接口

本节介绍 PS 端 C/C++ 程序中访问自定义算子寄存器和 PL DDR 的常用 API，包括读写寄存器、分配 PL DDR 空间、读写 PL DDR 数据等接口。

### 读寄存器

```cpp
int value = device.defaultRegRegion().read(uint32_t address, bool relative);
```

`read()` 函数用于读取指定寄存器地址中的数据。

| 参数 / 返回值   | 类型         | 说明                                                          |
| ---------- | ---------- | ----------------------------------------------------------- |
| `address`  | `uint32_t` | 寄存器的访问地址，32 位。custom op 的寄存器地址空间为 `0x400C0000 ~ 0x400FFFFF` |
| `relative` | `bool`     | 访问地址是否为相对地址，默认值为 `false`                                    |
| `value`    | `int`      | 返回寄存器值，32 位                                                 |

### 写寄存器

```cpp
device.defaultRegRegion().write(uint32_t address, uint32_t value, bool relative);
```

`write()` 函数用于向指定寄存器地址写入数据。

| 参数         | 类型         | 说明                                                           |
| ---------- | ---------- | ------------------------------------------------------------ |
| `address`  | `uint32_t` | 寄存器的写访问地址，32 位。custom op 的寄存器地址空间为 `0x400C0000 ~ 0x400FFFFF` |
| `value`    | `uint32_t` | 寄存器的写值，32 位。将该值写入对应的寄存器                                      |
| `relative` | `bool`     | 访问地址是否为相对地址，默认值为 `false`                                     |

### 分配 PL DDR 地址空间

```cpp
auto mem_chunk = device.defaultMemRegion().malloc(
    uint64_t byte_size,
    bool auto_free,
    uint64_t alignment
);
```

执行 `malloc()` 函数后，`device` 将分配一段 PL DDR 空间供用户使用，用户可通过 `mem_chunk` 对该段空间进行读写访问。

| 参数          | 类型         | 说明                           |
| ----------- | ---------- | ---------------------------- |
| `byte_size` | `uint64_t` | 分配内存的大小                      |
| `auto_free` | `bool`     | 指定分配的内存是否会被自动释放，默认值为 `false` |
| `alignment` | `uint64_t` | 指定分配的内存地址对齐的字节数，默认值为 `64`    |

### 写 PL DDR

```cpp
mem_chunk.write(uint64_t dest_offset, char* src, uint64_t byte_size);
```

`write()` 函数用于将缓存中的数据写入 PL DDR。

| 参数            | 类型         | 说明                              |
| ------------- | ---------- | ------------------------------- |
| `dest_offset` | `uint64_t` | 数据被写入的偏移位置，相对于 `mem_chunk` 的首地址 |
| `src`         | `char*`    | 待写入的数据缓存                        |
| `byte_size`   | `uint64_t` | 写入数据的字节大小                       |

### 读 PL DDR

```cpp
mem_chunk.read(char* dest, uint64_t src_offset, uint64_t byte_size);
```

`read()` 函数用于从 PL DDR 中读取数据到指定缓存。

| 参数           | 类型         | 说明                                |
| ------------ | ---------- | --------------------------------- |
| `dest`       | `char*`    | 数据被读取到的缓存                         |
| `src_offset` | `uint64_t` | 从该偏移位置开始读取数据，相对于 `mem_chunk` 的首地址 |
| `byte_size`  | `uint64_t` | 读取数据的字节大小                         |
 
### PL DDR间拷贝
`void Plddr_memcpy(uint64_t read_bottom, uint64_t read_top, uint64_t write_bottom, uint64_t write_top, icraft::xrt::Device& device)`

- 参数:

  **read_bottom** –PLDDR上src的起始地址;

  **read_top** –PLDDR上src的结束地址;

  **write_bottom**–PLDDR上dest的起始地址;

  **write_top** –PLDDR上dest的结束地址;

  **device**– 设备对象；

- 返回:

  无返回值。

- 说明:

  PLDDRMemRegion::Plddr_memcpy()是将PLDDR上src的数据拷贝给PLDDR上dst的一个函数；需用户给定src存储在PLDDR上的起始&结束地址，以及需要将src拷贝到dest在PLDDR上的起始&结束地址。

- 注意：

  src和dest地址长度需一致，且必须是64整数倍

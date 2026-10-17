##### with
* 执行完之后就销毁示例不保存
    ```python
    with xxxxxxxxxxxxxxxxx as yyyy
        balabala
        shilashila
    other code
    ```
    * 执行完 with 之后的缩进行(balabala,shilashila)就退出，继续执行 other code
    * xxxxxxxxxxxxxxxxx 是实例化一个对象，yyyy 是名字控制这个对象
    * xxxxxxxxxxxxxxxxx 必须是一个支持上下文管理协议的对象




##### socket
* socket 的本意是“插座、插孔”，一次通信需要两端参与，每一端需要有一个地址：192。168.3.xxx:yyyy ，和一个接口 socket
* socket 可以绑定到ip端口上
* 代码示例
    ```python
    # 通过 IPV4 连接
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as connection:
        connection.connect(('192.168.1.10', 2000))
    ```
    ```python
    # 本机连接
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
    connection.connect(str(SOCKET_FILE))
    ```
    * AF = Address Family = 地址族类

        | 地址类型 | 用途 | 地址示例 |
        |---|---|---|
        | `AF_UNIX` | 本机进程通信 | `'/tmp/control.sock'` |
        | `AF_INET` | IPv4，本机或跨机器 | `('192.168.1.10', 2000)` |
        | `AF_INET6` | IPv6，本机或跨机器 | `('::1', 2000)` |
    
        | 类型 | 含义 | 特点 |
        |---|---|---|
        | `SOCK_STREAM` | 字节流 | 可靠、有序，不保留消息边界；常用于 TCP、UNIX socket |
        | `SOCK_DGRAM` | 数据报 | 保留每条消息的边界；常用于 UDP、UNIX socket |
        | `SOCK_SEQPACKET` | 有序分组 | 可靠、有序，同时保留消息边界；Linux UNIX socket 支持 |
        | `SOCK_RAW` | 原始套接字 | 操作较底层的网络报文，通常需要额外权限 |

    * 使用 AF_UNIX 本机连接时，需要在本地生成一个 .sock 文件，让通信两都能定位到，.sock 文件 类似地址簿，协调两端通信


##### try except
* 用来捕获并处理程序运行时的异常
* 基本语法
    ```python
        try:
            # 尝试执行的代码
            number = int('abc')
        except ValueError as error:
            # 发生指定异常时执行
            print(f'转换失败：{error}')
        except OSError as error:
            print(f'文件或通信错误：{error}')
    ```
    * ValueError, OSError 都是错误的类型，由 python 本身定义。
    * 没有异常：执行完 try，跳过 except。
    * 发生匹配到类型的异常：停止执行 try 中剩余的代码，进入 except。
    * 异常不匹配：继续向外抛出，由外层处理。
    * 如果不能确定可能的错误类型就写
        ```python
            try:
                # 可能发生错误的代码
                ...
            except Exception as error:
                print(f'错误类型：{type(error).__name__}')
                print(f'错误原因：{error}')
        ```
        可以捕获绝大多数错误类型

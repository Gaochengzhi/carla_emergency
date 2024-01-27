import multiprocessing
import time


class baseAgent:
    def __init__(self, name):
        self.name = name

    def rec(self):
        pass

    def send(self):
        pass

    def run(self):
        pass

# 创建一个子类，同时继承baseAgent和multiprocessing.Process


class MyAgent(baseAgent, multiprocessing.Process):
    def __init__(self, name):
        # 调用baseAgent的构造函数
        baseAgent.__init__(self, name)
        multiprocessing.Process.__init__(self)

    def run(self):
        # 重写run方法以执行特定的操作
        i = 0
        while True:
            i += 1
            print(f"{self.name} is running")
            self.name = self.name + str(i)
            time.sleep(1)

    def get_name(self):
        return self.name


# 创建MyAgent对象
agent = MyAgent("Agent1")


# 启动子进程
agent.start()

while True:
    print(agent.get_name())
    time.sleep(0.5)


# 等待子进程完成
# agent.join()

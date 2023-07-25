# -*- coding: utf-8 -*-
class ErrorAnalyzer:
    def __init__(self, error_range, num_judgments):
        self._error_range = error_range
        self._previous_data = 0
        self._consecutive_count = 0
        self._current_list = []
        self._num_judgments = num_judgments
        self._Fource_out = 0  # 强制退出位，如果这个值大于一定界限，则清空列表

    def analyze(self, data):
        current_data = data
        self._current_list.append(current_data)

        if self._previous_data != 0:
            if abs(current_data - self._previous_data) <= self._error_range:
                self._consecutive_count += 1
                if self._consecutive_count >= self._num_judgments:
                    # 重置元素，为下一次数据做准备
                    result = self._current_list[0]
                    self._consecutive_count = 0
                    # self._current_list.clear() # python3中才有clear
                    del self._current_list[:] # python2.7中清空列表的办法
                    self._previous_data = 0
                    self._Fource_out = 0
                    print("误差在允许范围内……")
                    return result, True  # 返回最初的值
            else:
                self._consecutive_count = 0
                self._Fource_out += 1
                if self._Fource_out >= 4:
                    self._consecutive_count = 0
                    # self._current_list.clear()
                    del self._current_list[:]
                    self._previous_data = 0
                    self._Fource_out = 0
                    print("误差不符合要求……")
        else:
            self._previous_data = current_data
        #
        return None, False

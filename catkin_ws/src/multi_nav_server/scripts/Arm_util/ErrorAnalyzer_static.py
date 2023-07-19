#!/usr/bin/env python
# -*- coding: utf-8 -*-
class ErrorAnalyzer_static:
    def __init__(self, error_range, num_judgments_static):
        self.error_range = error_range
        self.previous_data = 0
        self.consecutive_count = 0
        self.current_list = []
        self.num_judgments_static = num_judgments_static
        self.Fource_out = 0  # 强制退出位，如果这个值大于一定界限，则清空列表

    def analyze_static_method(self, data):
        current_data = data
        self.current_list.append(current_data)

        if self.previous_data != 0:
            if abs(current_data - self.previous_data) <= self.error_range:
                self.consecutive_count += 1
                if self.consecutive_count >= self.num_judgments_static:
                    # 重置元素，为下一次数据做准备
                    result = self.current_list[0]
                    self.consecutive_count = 0
                    self.current_list.clear()
                    self.previous_data = 0
                    self.Fource_out = 0
                    print("误差在允许范围内……")
                    return result, True  # 返回最初的值
            else:
                self.consecutive_count = 0
                self.Fource_out += 1
                if self.Fource_out >= 4:
                    self.consecutive_count = 0
                    self.current_list.clear()
                    self.previous_data = 0
                    self.Fource_out = 0
                    print("误差不符合要求……")
        else:
            self.previous_data = current_data
        #
        return None, False

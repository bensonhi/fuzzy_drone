import skfuzzy as fuzzy
import numpy as np
import skfuzzy.control as ctrl

# 定義模糊邏輯控制器
class FuzzyController:
    def __init__(self):
        self.engine = ctrl.ControlSystem()

        # 定義模糊變數
        self.vertical_error = ctrl.Antecedent(np.arange(-100, 101, 1), 'vertical_error')
        self.horizontal_error = ctrl.Antecedent(np.arange(-100, 101, 1), 'horizontal_error')

        self.vertical_motor = ctrl.Consequent(np.arange(-100, 101, 1), 'vertical_motor')
        self.horizontal_motor = ctrl.Consequent(np.arange(-100, 101, 1), 'horizontal_motor')

        # 定義模糊集合和規則
        self.vertical_error['negative'] = fuzzy.trimf(self.vertical_error.universe, [-100, -100, 0])
        self.vertical_error['zero'] = fuzzy.trimf(self.vertical_error.universe, [-10, 0, 10])
        self.vertical_error['positive'] = fuzzy.trimf(self.vertical_error.universe, [0, 100, 100])

        self.horizontal_error['negative'] = fuzzy.trimf(self.horizontal_error.universe, [-100, -100, 0])
        self.horizontal_error['zero'] = fuzzy.trimf(self.horizontal_error.universe, [-10, 0, 10])
        self.horizontal_error['positive'] = fuzzy.trimf(self.horizontal_error.universe, [0, 100, 100])

        self.vertical_motor['negative'] = fuzzy.trimf(self.vertical_motor.universe, [-100, -100, 0])
        self.vertical_motor['zero'] = fuzzy.trimf(self.vertical_motor.universe, [-10, 0, 10])
        self.vertical_motor['positive'] = fuzzy.trimf(self.vertical_motor.universe, [0, 100, 100])

        self.horizontal_motor['negative'] = fuzzy.trimf(self.horizontal_motor.universe, [-100, -100, 0])
        self.horizontal_motor['zero'] = fuzzy.trimf(self.horizontal_motor.universe, [-10, 0, 10])
        self.horizontal_motor['positive'] = fuzzy.trimf(self.horizontal_motor.universe, [0, 100, 100])

        # 定義規則
        rule1 = ctrl.Rule((self.vertical_error['negative'] & self.horizontal_error['negative']), (self.vertical_motor['positive'] ,self.horizontal_motor['positive']), label="1")
        rule2 = ctrl.Rule((self.vertical_error['zero'] & self.horizontal_error['negative']), (self.vertical_motor['zero'] , self.horizontal_motor['positive']), label="2")
        rule3 = ctrl.Rule((self.vertical_error['positive'] & self.horizontal_error['negative']), (self.vertical_motor['negative'], self.horizontal_motor['positive']), label="3")
        rule4 = ctrl.Rule((self.vertical_error['negative'] & self.horizontal_error['zero']), (self.vertical_motor['positive'] , self.horizontal_motor['zero']), label="4")
        rule5 = ctrl.Rule((self.vertical_error['zero'] & self.horizontal_error['zero']), (self.vertical_motor['zero'], self.horizontal_motor['zero']), label="5")
        rule6 = ctrl.Rule((self.vertical_error['positive'] & self.horizontal_error['zero']), (self.vertical_motor['negative'], self.horizontal_motor['zero']), label="6")
        rule7 = ctrl.Rule((self.vertical_error['negative'] & self.horizontal_error['positive']), (self.vertical_motor['positive'], self.horizontal_motor['negative']), label="7")
        rule8 = ctrl.Rule((self.vertical_error['zero'] & self.horizontal_error['positive']), (self.vertical_motor['zero'], self.horizontal_motor['negative']), label="8")
        rule9 = ctrl.Rule((self.vertical_error['positive'] & self.horizontal_error['positive']), (self.vertical_motor['negative'] , self.horizontal_motor['negative']), label="9")

        # 將規則加入模糊引擎
        self.engine = ctrl.ControlSystem(rules=[rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9])

    def compute(self, vertical_error, horizontal_error):
        # 建立模糊控制系統
        control_system = ctrl.ControlSystemSimulation(self.engine)

        # 設定輸入
        control_system.input['vertical_error'] = vertical_error
        control_system.input['horizontal_error'] = horizontal_error

        # 執行模糊推論
        control_system.compute()

        # 取得輸出
        vertical_motor = control_system.output['vertical_motor']
        horizontal_motor = control_system.output['horizontal_motor']

        return vertical_motor, horizontal_motor

    def run(self,target_box_central,camera_size):
        # 計算目標位置相對於攝像頭中心的偏移量
        vertical_error = (camera_size[1] / 2) - target_box_central[1]
        horizontal_error = target_box_central[0] - (camera_size[0] / 2)

        vertical_error= vertical_error*100/(camera_size[1]/2)
        horizontal_error = horizontal_error * 100 / (camera_size[0]/2)

        # 使用模糊控制器計算電機控制值
        vertical_motor, horizontal_motor = self.compute(vertical_error, horizontal_error)

        # 輸出結果
        print("Vertical Motor:", vertical_motor)
        print("Horizontal Motor:", horizontal_motor)

# coding=utf-8
from typing import Optional, List, Tuple
'''
工作台类
'''

import logging

# logging.basicConfig(filename='log.log', level=logging.DEBUG)
class Workbench:
    ITEMS_BUY = [0, 3000, 4400, 5800, 15400, 17200, 19200, 76000]  # 每个物品的购买价
    ITEMS_SELL = [0, 6000, 7600, 9200, 22500, 25000, 27500, 105000]
    ITEMS_NEED = [[] for _ in range(8)]  # 记录收购每个商品的工作台编号
    WORKSTAND_IN = {1: [], 2: [], 3: [], 4: [1, 2], 5: [1, 3],
                    6: [2, 3], 7: [4, 5, 6], 8: [7], 9: list(range(1, 8))}
    WORKSTAND_OUT = {i: i for i in range(1, 8)}
    WORKSTAND_OUT[8] = None
    WORKSTAND_OUT[9] = None
    WORKSTAND_FULL = {  # 记录每种工作台材料格满的情况
        4: sum([1 << i for i in (1, 2)]),
        5: sum([1 << i for i in (1, 3)]),
        6: sum([1 << i for i in (2, 3)]),
        7: sum([1 << i for i in (4, 5, 6)])
    }

    def __init__(self, ID: int, typeID: int, loc: Tuple[float]):
        self.ID = ID  # 编号
        self.typeID = typeID  # 类型
        self.loc = loc  # 位置
        self.product_time = 0  # 剩余生产时间
        self.material = 0  # 原材料格状态
        self.product_status = 0  # 产品格状态
        self.material_pro = 0  # 原料格预定状态, 防止有多个机器人将其作为出售目标
        self.product_pro = 0  # 防止有多个机器人将其作为购买目标
        self.target_workbench_list = []  # 收购此工作台的产品且可到达的工作台列表
        self.buy_price = self.ITEMS_BUY[self.typeID] if self.typeID < len(self.ITEMS_BUY) else 0  # 进价  
        self.sell_price = self.ITEMS_SELL[self.typeID] if self.typeID < len(self.ITEMS_SELL) else 0  # 售价
            

    def get_materials_num(self):  # 返回格子数目
        num = 0
        v = self.material
        while v:
            num += v & 1
            v >>= 1
        return num

    def check_materials_full(self) -> bool:
        '''
        检测原料格是否已满
        '''
        if self.typeID not in self.WORKSTAND_FULL:
            return True
        return self.material == self.WORKSTAND_FULL[self.typeID]

    def check_material(self, mateial_ID: int) -> bool:
        '''
        检测原材料是否已放置
        True表示已有材料
        '''
        return 1 << mateial_ID & self.material != 0

    def check_material_pro(self, mateial_ID: int) -> bool:
        '''
        检测原材料格子是否已被预订
        True表示已有材料
        '''
        return 1 << mateial_ID & self.material_pro != 0

    def pro_sell(self,  mateial_ID: int, sell=True) -> bool:
        '''
        预售接口 如果是89特殊处理
        mateial_ID: 传入要出售商品的类型ID
        sell: True表示预售, False表示取消预售
        设置成功返回True
        '''
        if self.typeID in [8, 9]:
            return True
        if sell:
            if self.check_material_pro(mateial_ID):
                # 已被预定
                return False
            self.material_pro += 1 << mateial_ID
        elif self.check_material_pro(mateial_ID):
            self.material_pro -= 1 << mateial_ID
        return True

    def pro_buy(self, buy=True):
        '''
        预购接口, 如果是123特殊处理
        sell: True表示预购, False表示取消预购 
        设置成功返回True    
        '''
        if self.typeID in [1, 2, 3]:
            return True
        if buy:
            if self.product_pro:
                return False
            self.product_pro = 1
        else:
            self.product_pro = 0
        return True

    def update(self, s: str):
        # 根据判题器传来的状态修订本机状态
        # logging.info(f'本地工作台坐标：{self.loc} 判题器坐标f{s.split()[1:3]}')
        self.product_time, self.material, self.product_status = map(int, s.split()[3:])

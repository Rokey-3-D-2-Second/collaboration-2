from DR_common2 import posx

import time

from util import config

class Mover:
    white_board_home = posx([0, 0, 0, 0, 0, 0])
    pen_holder = posx([2.970, 574.210, -136.250, 90.0, 88.17, -91.58])
    eraser_holder = posx([145.0, 615.0, -155.0, 90.0, 90.0, -90.0])

    holder_mode = {
        1: pen_holder,
        2: eraser_holder
    }
    
    def set_dependencies(
            self,

            movel, 

            get_current_posx,
        ):

        self.movel = movel
        self.get_current_posx = get_current_posx

    def move_to_home(self):
        self.movel(self.white_board_home, config.VEL, config.ACC)

    def move_to_start(self, traj):
        self.movel(traj[0], config.VEL, config.ACC)

    def move_to_holder(self, mode):
        self.movel(self.holder_mode[mode], config.VEL, config.ACC)

    def move_to_above_holder(self, mode):
        self.holder_mode[mode][1] -= 30
        self.move_to_holder(mode)

    def down_to_hold(self, mode):
        self.holder_mode[mode][1] += 30
        self.move_to_holder(mode)

    def up_from_holder(self, mode):
        self.holder_mode[mode][1] -= 20
        self.move_to_holder(mode)
    
    def down_to_release(self, mode):
        self.holder_mode[mode][1] += 18
        self.move_to_holder(mode)
        self.holder_mode[mode][1] += 2

    def pen_down(self, callback):
        """로봇 펜을 보드(종이)로 누르는 동작을 수행합니다."""
        callback()

    def pen_up(self, callback):
        """로봇 펜을 보드(종이)에서 들어올립니다."""
        current_posx = self._get_cur_posx()[0]
        current_posx[2] -= 5
        callback()
        time.sleep(0.1)
        self.movel(current_posx, config.VEL, config.ACC)

    def _get_cur_posx(self):
        """
        현재 로봇의 작업 좌표계(Task Space Position, posx)를 읽어오는 함수.

        주요 동작:
            - 최대 5초 동안 반복해서 get_current_posx()를 호출하여 posx 정보를 시도한다.
            - IndexError가 발생하면 0.1초 후 재시도한다.
            - 5초 이내에 posx를 정상적으로 얻으면 해당 값을 반환한다.
            - 5초가 지나도 값을 얻지 못하면 오류 로그를 남기고, 모든 값이 0인 posx([0,0,0,0,0,0])를 반환한다.

        Returns:
            posx: 6차원 작업좌표(posx) 리스트 객체 또는 실패시 [0,0,0,0,0,0]
        """
        start = time.time()
        while time.time() - start < 5:
            try:
                cur_posx = self.get_current_posx()
                return cur_posx
            except IndexError as e:
                time.sleep(0.1)
                continue
        return posx([0,0,0,0,0,0])
from DR_common2 import posx

import time

from util import config

class Mover:
    home = posx([372.49, 16.510, 170.42, 90.0, 180.0, 180.0])
    tray = posx([572.49, 16.510, 70.42, 90.0, 180.0, 180.0])
    
    def set_dependencies(
            self,

            movel, 

            get_current_posx,
        ):

        self._movel = movel
        self._get_current_posx = get_current_posx

    def move_to_home(self):
        self._movel(self.home, config.VEL, config.ACC)

    def move_to_tray(self):
        self._movel(self.tray, config.VEL, config.ACC)

    def move_to_target(self, target):
        self._movel(target, config.VEL, config.ACC)

    def _down(self):
        """로봇을 target으로 내리는 동작을 수행합니다."""
        current_posx = self._get_cur_posx()[0]
        current_posx[2] -= 10
        self._movel(current_posx, config.VEL, config.ACC)

    def _up(self):
        """로봇을 target에서 들어올립니다."""
        current_posx = self._get_cur_posx()[0]
        current_posx[2] += 5
        self._movel(current_posx, config.VEL, config.ACC)

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
                cur_posx = self._get_current_posx()
                return cur_posx
            except IndexError as e:
                time.sleep(0.1)
                continue
        return posx([0,0,0,0,0,0])
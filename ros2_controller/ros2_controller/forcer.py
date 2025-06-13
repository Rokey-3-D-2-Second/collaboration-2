import time

class Forcer:
    def set_dependencies(
            self,

            check_force_condition,
            task_compliance_ctrl,
            release_compliance_ctrl,
            set_desired_force,
            release_force,

            DR_AXIS_Z,

            DR_FC_MOD_REL
        ):

        self._task_compliance_ctrl = task_compliance_ctrl
        self._set_desired_force = set_desired_force
        self._release_force = release_force
        self._release_compliance_ctrl = release_compliance_ctrl
        self._check_force_condition = check_force_condition
        self._DR_AXIS_Z = DR_AXIS_Z
        self._DR_FC_MOD_REL = DR_FC_MOD_REL

    def force_on(self, fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0]):
        """로봇의 힘/컴플라이언스(유연제어) 제어를 모두 설정합니다."""
        self._task_compliance_ctrl()
        time.sleep(0.1)
        self._set_desired_force(fd=fd, dir=dir, mod=self._DR_FC_MOD_REL)

    def force_off(self):
        """로봇의 힘/컴플라이언스(유연제어) 제어를 모두 해제합니다."""
        self._release_force()
        self._release_compliance_ctrl()

    def check_touch(self):
        """Z축 힘(접촉력)이 5~21 사이가 될 때까지 대기"""
        # if mode == 2: fd[2] = 5 # erase mode
        while self._check_force_condition(self._DR_AXIS_Z, min=5, max=21): pass
        # self.release_force()
        # self.set_desired_force(fd=fd, dir=dir, mod=self.DR_FC_MOD_REL)
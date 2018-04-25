class StateMachine:
    def __init__(self):
        self._states = {}
        self.init_states()
        self._state = self.state_on_init
        self._state.on_state_enable()

    def init_states(self):
        pass

    @property
    def state_on_init(self):
        pass

    def switch_state_to(self, state):
        if self._state != state:
            self._state.on_state_disable()
            self._state = state
            self._state.on_state_enable()

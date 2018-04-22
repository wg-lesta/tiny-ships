class StateMachine:
    def __init__(self):
        self._states = {}
        self._init_states()
        self._state = self._state_on_init()
        self._state.on_state_enable()

    def _init_states(self):
        pass

    def _state_on_init(self):
        pass

    def _switch_state_to(self, state):
        if self._state != state:
            self._state.on_state_disable()
            self._state = state
            self._state.on_state_enable()

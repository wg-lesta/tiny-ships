
class CarrierState:
    SEND_ACTION = 'send_plane'

    def __init__(self, ship):
        self.ship = ship

    def on_state_enable(self):
        pass

    def on_state_disable(self):
        pass

    def update(self, keys):
        pass


class CarrierSendState(CarrierState):
    def __init__(self, ship):
        CarrierState.__init__(self, ship)

    def update(self, keys):
        if CarrierState.SEND_ACTION in keys:
            if self.ship.can_send_plane():
                self.ship.send_plane()
                self.ship.to_idle_state()


class CarrierIdleState(CarrierState):
    def __init__(self, ship):
        CarrierState.__init__(self, ship)
        self.ticks = 0

    def on_state_enable(self):
        self.ticks = 0

    def update(self, keys):
        from entities.carrier import Carrier
        self.ticks += 1
        if self.ticks >= Carrier.SEND_INTERVAL:
            self.ship.to_send_state()

from ADRC_controller import ADRC_Controller

class adrc_attitude_controller:
    def __init__(self, dt, phi_para, theta_para, psi_para):
        self.dt = dt
        self.phi_controller = ADRC_Controller(dt, phi_para['L'], phi_para['b0'], phi_para['wc'], phi_para['td_flag'])
        self.theta_controller = ADRC_Controller(dt, theta_para['L'], theta_para['b0'], theta_para['wc'], theta_para['td_flag'])
        self.psi_controller = ADRC_Controller(dt, psi_para['L'], psi_para['b0'], psi_para['wc'], psi_para['td_flag'])
    
    def calculate_output(self):
        
        pass
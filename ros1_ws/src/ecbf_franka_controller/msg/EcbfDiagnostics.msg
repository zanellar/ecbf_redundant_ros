# Fixed-size real-time diagnostics produced by EcbfController.
std_msgs/Header header

# CBF status codes mirrored from ecbf_franka_controller::CbfStatus.
uint8 CBF_DISABLED=0
uint8 CBF_INACTIVE=1
uint8 CBF_BOUNDED=2
uint8 CBF_ACTIVE=3
uint8 CBF_INFEASIBLE_ZERO_GRADIENT=4
uint8 CBF_INFEASIBLE_BOUNDS=5
uint8 CBF_NUMERICAL_FAILURE=6

float64 elapsed
float64 alpha
int32 cbf_type
bool cbf_enabled
bool cbf_applied
bool cbf_active
bool cbf_feasible
uint8 cbf_status

float64 barrier_function_value
float64 cbf_constraint_nom_value
float64 cbf_constraint_safe_value
float64 cbf_correction_norm

float64 total_energy
float64 operational_energy
float64 selected_energy
float64 directional_velocity
float64 directional_inertia

float64[7] q
float64[7] dq
float64[7] tau_nominal
float64[7] tau_safe
float64[7] tau_residual_command
float64[7] tau_command
float64[7] coriolis
float64[3] position
float64[6] task_velocity

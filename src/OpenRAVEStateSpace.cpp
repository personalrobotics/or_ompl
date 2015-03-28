#include <algorithm> // for std::maximum_element
#include <cstring> // for std::memcpy
#include <functional> // for std::times
#include <numeric> // for std::accumulate
#include <openrave/openrave.h>

typedef ompl::base::RealVectorStateSpace::StateType ORState;

class OpenRAVEStateSpace : public ompl::base::StateSpace {
public:
    OpenRAVEStateSpace(OpenRAVE::PlannerBase::PlannerParametersConstPtr const &params);

    virtual unsigned int getDimension() const;
    virtual double getMaximumExtent() const;
    virtual double getMeasure() const;

    virtual void enforceBounds(State *state);
    virtual bool satisfiesBounds(State const *state);

    virtual void copyState(State *destination, State const *source) const;

    virtual double distance(const State *state1, State const *state2) const;

    virtual unsigned int getSerializationLength() const;
    virtual void serialize(void *serialization, State const *state) const;
    virtual void deserialize(State *state, void const *serialization) const;

    virtual bool equalStates(State const *state1, State const *state2) const;
    virtual void interpolate(State const *from, State const *to,
                             double const t, State *state) const;

    virtual StateSamplerPtr allocDefaultStateSampler() const;
    virtual StateSamplerPtr allocStateSampler() const;

    void setStateSamplerAllocator(StateSamplerAllocator const &ssa);
    void clearStateSamplerAllocator();

    virtual State *allocState() const;
    virtual void freeState(State *state) const;

private:
    OpenRAVE::PlannerBase::PlannerParametersConstPtr params_;
    std::vector<double> extents_;
    std::vector<bool> is_circular_;
};

OpenRAVEStateSpace::OpenRAVEStateSpace(
        OpenRAVE::PlannerBase::PlannerParametersConstPtr const &params)
    : params_(params),
{
    params_->Validate();

    extents_.resize(getDimension());
    is_circular_.resize(getDimension());

    for (size_t i = 0; i < getDimension(); ++i) {
        // TODO: Infer which joints are circular.
        is_circular[i] = false;

        if (is_circular_[i]) {
            extents_[i] = 2 * M_PI;
        } else {
            extents_[i] = params_->_vConfigUpperLimit[i]
                   - params_->_vConfigLowerLimit[i];
        }
    }
}

unsigned int OpenRAVEStateSpace::getDimension() const
{
    return params->_configurationspecification.GetDOF();
}

double OpenRAVEStateSpace::getMaximumExtent() const
{
    return *std::max_element(extents_.begin(), extents_.end();
}

double OpenRAVEStateSpace::getMeasure() const
{
    return std::accumulate(extents_.begin(), extents_.end(),
                           std::multiplies<double>);
}

void OpenRAVEStateSpace::enforceBounds(State *state)
{
    ORState *or_state = state->as<ORState>();

    for (size_t i = 0; i < getDimension(); ++i) {
        double &value = or_state->values[i];
        double const lower_limit = params_->_vConfigLowerLimit[i];
        double const upper_limit = params_->_vConfigUpperLimit[i];

        if (is_circular_[i]) {
            // Do nothing. Circular joints have no bounds.
        } else if (value < lower_limit) {
            value = lower_limit;
        } else if (value > upper_limit) {
            value = upper_limit;
        }
    }
}

bool OpenRAVEStateSpace::satisfiesBounds(State const *state)
{
    ORState const *or_state = state->as<ORState>();

    for (size_t i = 0; i < getDimension(); ++i) {
        double const &value = or_state->values[i];

        if (!is_circular_[i] && (value < params_->_vConfigLowerLimit[i]
                             ||  value > params_->_vConfigUpperLimit[i]) {
            return false; // Exceeds the upper limit.
        }
    }

    return true;
}

unsigned int OpenRAVEStateSpace::getSerializationLength() const
{
    return getDimension() * sizeof(double);
}

void OpenRAVEStateSpace::serialize(void *serialization, State const *state) const
{
    std::memcpy(serialization, state->as<ORState>->values,
                getSerializationLength());
}

void OpenRAVEStateSpace::deserialize(State *state,
                                     void const *serialization) const
{
    std::memcpy(state->as<ORState>->values, serialization,
                getSerializationLength());
}

bool OpenRAVEStateSpace::equalStates(State const *state1,
                                     State const *state2) const
{
    ORState const *or_state1 = state1->as<ORState>();
    ORState const *or_state2 = state2->as<ORState>();

    for (size_t i = 0; i < getDimension(); ++i) {
        OpenRAVE::dReal diff = std::fabs(or_state1->values[i]
                                       - or_state2->values[i]);

        // TODO: Should we use DOF resolution for this comparison?
        // TODO: Should we consider wrapping circular joints?

        if (diff > params_->_vConfigResolution[i]) {
            return false; 
        }
    }

    return true;
}

void OpenRAVEStateSpace::interpolate(State const *from, State const *to,
                                     double const t, State *state) const
{
    ORState const *or_from  = from->as<ORState>();
    ORState const *or_to    = to->as<ORState>();
    ORState       *or_state = state->as<ORState>();

    for (size_t i = 0; i < getDimension(); ++i) {
        // Copy the values to avoid aliasing issues.
        double const from_value = or_from->values[i];
        double const to_value = or_to->values[i];

        // TODO: Handle circular joints here.
        double const step = to_value - from_value;
        or_state->values[i] = from_value + t * step;
    }
}

StateSamplerPtr allocDefaultStateSampler() const
{
    throw std::runtime_error("not implemented");
}

StateSamplerPtr allocStateSampler() const
{
    throw std::runtime_error("not implemented");
}

void setStateSamplerAllocator(StateSamplerAllocator const &ssa)
{
    throw std::runtime_error("not implemented");
}

void clearStateSamplerAllocator()
{
    throw std::runtime_error("not implemented");
}

State *OpenRAVEStateSpace::allocState() const
{
    return new ORState;
}

void OpenRAVEStateSpace::freeState(State *state) const
{
    delete state->as<ORState>();
}


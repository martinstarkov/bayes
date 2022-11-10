// TODO: Add header include guards.



// TODO: Make filter actually return something.
// Currently filter acts as a empty pass-through container.
class Filter {
public:
    Filter() = default;
    Filter(float value) : value{ value }, filtered_value{ value } {}
    void UpdateValue(float value) {
        this->value = value;
        // TODO: Replace this.
        this->filtered_value = this->value;
    }
    float GetValue() {
        return filtered_value;
    }
private:
    float value = 0.0f;
    float filtered_value = 0.0f;
    float integral = 0.0f;
};
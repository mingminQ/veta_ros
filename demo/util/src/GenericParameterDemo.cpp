#include "veta/util/generic/GenericParameter.h"
#include "veta/util/LogHandler.h"

#include "veta_noetic/util/Timer.h"
#include "ros/ros.h"

class MyClass
{
// "MyClass" member variables
public:

    // "MyClass" parameter key
    enum ParameterKey
    {
        DOUBLE_PARAM = 0,
        INT_PARAM    = 1,
        STRING_PARAM = 2,
        UNKNOWN_PARAM

    }; // enum ParameterKey

private:

    // Parameter container
    veta::GenericParameter m_parameter;

    /** @brief Parameter decleration function */
    void declareParameter();

// "MyClass" member functions
public:

    /** @brief Class constructor */
    MyClass();

    /** @brief Class destructor */
    ~MyClass() = default;

    /** @brief Parameter reference access function */
    veta::GenericParameter &parameter();

    /** @brief Parameter printer function */
    void printParameter();
    
}; // class MyClass

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "GenericParameterDemo");
    ros::NodeHandle nh;
    
    TIMER_START;

    // default parameter
    MyClass cls;
    //cls.printParameter();

    // parameter set : invalid range
    cls.parameter().set(MyClass::DOUBLE_PARAM, -99.0, MyClass::INT_PARAM, 1, MyClass::STRING_PARAM, std::string("D"));
    cls.printParameter();

    // parameter set : valid range
    cls.parameter().set(MyClass::DOUBLE_PARAM, 5.0, MyClass::INT_PARAM, 2, MyClass::STRING_PARAM, std::string("C"));
    cls.printParameter();

    TIMER_END;

    return 0;
}

/** @brief Class constructor */
MyClass::MyClass()
{
    declareParameter();
}

/** @brief Parameter decleration function */
void MyClass::declareParameter()
{
    // double parameter : boundary
    m_parameter.declare<double>(DOUBLE_PARAM, "DOUBLE_PARAM", 0.0, -10.0, 10.0);

    // int parameter : function
    m_parameter.declare<int>(INT_PARAM, "INT_PARAM", 0, [](const int &value){return value % 2 == 0;});

    // string parameter : list
    m_parameter.declare<std::string>(STRING_PARAM, "STRING_PARAM", std::string("A"), {std::string("A"), std::string("B"), std::string("C")});
}

/** @brief Parameter reference access function */
void MyClass::printParameter()
{
    // double parameter
    VETA_INFO("DOUBLE_PARAM  >> %lf", m_parameter.get<double>(DOUBLE_PARAM));
    VETA_INFO("INT_PARAM     >> %d" , m_parameter.get<int>(INT_PARAM));
    VETA_INFO("STRING_PARAM3 >> %s" , m_parameter.get<std::string>(STRING_PARAM).c_str());
    std::cout << "---" << std::endl;
}

/** @brief Parameter printer function */
veta::GenericParameter &MyClass::parameter()
{
    return m_parameter;
}
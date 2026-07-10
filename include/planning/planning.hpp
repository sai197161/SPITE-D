#ifndef MY_CLASS_HPP
#define MY_CLASS_HPP

#include <string>

namespace MyProject {

    class MyClass {
    public:
        // Lifecycle
        MyClass();
        explicit MyClass(std::string name);
        ~MyClass();

        // Rule of 5 (Delete or implement explicitly if managing resources)
        MyClass(const MyClass& other) = default;
        MyClass(MyClass&& other) noexcept = default;
        MyClass& operator=(const MyClass& other) = default;
        MyClass& operator=(MyClass&& other) noexcept = default;

        // Getters / Setters
        [[nodiscard]] std::string getName() const;
        void setName(const std::string& name);

        // Business Logic
        void processData();

    private:
        std::string m_name;
    };

} // namespace MyProject

#endif // MY_CLASS_HPP

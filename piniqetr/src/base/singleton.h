#ifndef GLOBAL_SINGLETON_H_
#define GLOBAL_SINGLETON_H_

#if defined(DISALLOW_COPY_AND_ASSIGN)
#undef DISALLOW_COPY_AND_ASSIGN
#endif

#if defined(DECLARE_SINGLETON)
#undef DECLARE_SINGLETON
#endif

#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&) = delete;      \
  TypeName& operator=(const TypeName&) = delete

#define DECLARE_SINGLETON(classname)              \
 public:                                          \
  static classname* Instance() {                  \
    static classname* instance = new classname(); \
    return instance;                              \
  }                                               \
                                                  \
  DISALLOW_COPY_AND_ASSIGN(classname);            \
                                                  \
 private:                                         \
  classname();

#endif

#ifndef TRUNCATED_SVD_SOLVER_CHOLMOD_HELPERS_H
#define TRUNCATED_SVD_SOLVER_CHOLMOD_HELPERS_H

namespace truncated_svd_solver {

inline void deleteCholdmodPtr(cholmod_sparse* & ptr,
                              cholmod_common& cholmod) {
  if (ptr) {
    cholmod_l_free_sparse(&ptr, &cholmod);
  }
}
inline void deleteCholdmodPtr(cholmod_dense* & ptr,
                              cholmod_common& cholmod) {
  if (ptr) {
    cholmod_l_free_dense(&ptr, &cholmod);
  }
}

template<typename T>
struct SelfFreeingCholmodPtr {
  explicit SelfFreeingCholmodPtr(T* ptr,
                                 cholmod_common& cholmod)
      : ptr_(ptr),
        cholmod_(cholmod) {}

  ~SelfFreeingCholmodPtr() {
    reset(NULL);
  }

  void reset(T* ptr) {
    deleteCholdmodPtr(ptr_, cholmod_);
    ptr_ = ptr;
  }

  SelfFreeingCholmodPtr & operator=(T* ptr) {
    reset(ptr);
    return *this;
  }

  operator T*() {
    return ptr_;
  }

  T* operator->() {
    return ptr_;
  }

  T** operator&() {
    return &ptr_;
  }

 private:
  cholmod_common & cholmod_;
  T* ptr_;
};

}  // namespace truncated_svd_solver

#endif // TRUNCATED_SVD_SOLVER_TSVD_SOLVER_H

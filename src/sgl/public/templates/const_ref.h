#pragma once

/**
 * The idea is to use const reference in function arguments
 * only if we actually benefit from it, i.e. if we have to
 * copy a large amount of memory
 */
template<typename T, bool = (sizeof(T) > sizeof(void*))>
struct ConstRef { using Type = const T&; };

template<typename T>
struct ConstRef<T, false> { using Type = T; };

/// Quick macro
#define ConstRefT(T) typename ConstRef<T>::Type
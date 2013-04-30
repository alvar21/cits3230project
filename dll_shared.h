/// Contains data types and definitions shared by the data link layers.

#ifndef DLL_SHARED_H
#define DLL_SHARED_H

#include <stddef.h>

#define DLL_MTU 8192 // The maximum size of any data link layer frame.

/// Defines the type of callback functions used by the data link layers to send
/// frame data up to the next layer.
///
typedef void (*up_from_dll_fn_ty)(int link, char const *data, size_t length);

#endif // DLL_SHARED_H

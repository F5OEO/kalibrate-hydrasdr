/**
 * @file circular_buffer.h
 * @brief High-performance Ring Buffer using Virtual Memory tricks.
 * 
 * Implements a "Magic Ring Buffer" where the same physical memory is mapped 
 * twice contiguously in virtual address space. This allows contiguous reads/writes 
 * across the boundary without manual wrapping logic.
 */

/*
 * @author Joshua Lackey (original)
 * @author Benjamin Vernoux <bvernoux@hydrasdr.com> (improvements)
 * @copyright 2010 Joshua Lackey, 2025 Benjamin Vernoux
 * @license BSD-2-Clause
 */
#ifndef __CIRCULAR_BUFFER_H__
#define __CIRCULAR_BUFFER_H__

#include <mutex> // Replaces pthread.h

#ifdef _WIN32
#include <windows.h>
#endif

class circular_buffer {
public:
	circular_buffer(unsigned int buf_len, unsigned int obj_size, int overwrite = 0);
	~circular_buffer();

	unsigned int write(const void *s, unsigned int len);
	unsigned int read(void *s, unsigned int len);
	void *peek(unsigned int *len);
	unsigned int purge(unsigned int len);
	unsigned int buf_len();
	unsigned int data_available();
	unsigned int space_available();
	unsigned int capacity();
	void flush();

private:
	char *m_buf;
	unsigned int m_r, m_w;
	unsigned int m_buf_len;   
	unsigned int m_item_size; 
	unsigned int m_buf_size;  
	int m_overwrite;
	
	// Thread safety: C++ Standard Mutex
	std::mutex m_mutex;

#ifdef _WIN32
	HANDLE d_handle;
	LPVOID d_first_copy;
	LPVOID d_second_copy;
#else
	int m_shm_fd;
	void *m_base;
#endif
};

#endif // __CIRCULAR_BUFFER_H__
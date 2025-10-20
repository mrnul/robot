#include <deque>

using std::deque;

template<typename T>
class CircularBuffer : public deque<T>
{
private:
	const int N;
public:
	CircularBuffer(const int N) : N(N), deque<T>(N)
	{
	}
	void insert(const T& item)
	{
		while (this->size() >= N)
		{
			this->pop_back();
		}
		this->emplace_front(item);
	}
};

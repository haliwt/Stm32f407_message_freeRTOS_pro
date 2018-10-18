#include <stdio.h>
#include <stdlib.h>
#include "avltree/avltree.h"
#include "avl_queue/avl_queue.h"

AVLtree *find(myType data, AVLtree *T)
{
	if(T == NULL)
		return NULL;
	if(data < T->element)
		return find(data, T->lchild);
	else if(data > T->element)
		return find(data, T->rchild);
	else
		return T;
}

AVLtree *findMin(AVLtree *T)
{
	if(T == NULL)
		return NULL;
	else if(T->lchild == NULL)
		return T;
	else
		return findMin(T->lchild);
}
AVLtree *findMax(AVLtree *T)
{
	if(T != NULL)
		while(T->rchild != NULL)
			T = T->rchild;
	return T;

}

int getHeight(AVLtree *T)
{
	return GET_HEIGHT(T);
}
/***************************************************
 *
 *函数名：
 *函数功能：LL -旋转 *T 父结点= K1 ,TT 右数子结点=K2
 *参数：
 *返回值：返回根节点
 *
****************************************************/
static AVLtree *left_left_rotation(AVLtree *T)
{
	AVLtree *TT;

	TT = T->lchild;
	T->lchild = TT->rchild;
	TT->rchild = T;

	T->height = MAX(getHeight(T->lchild), getHeight(T->rchild)) + 1; //WT.EDIT
	TT->height = MAX(getHeight(TT->lchild), T->height) + 1;  //WT.EDIT

	return TT;
}
/***************************************************
 *
 *函数名：
 *函数功能：RR -旋转 *T 父结点= K1 ,TT 右数子结点=K2
 *参数：
 *返回值：返回根节点
 *
****************************************************/
static AVLtree *right_right_rotation(AVLtree *T)
{
	AVLtree *TT;

	TT = T->rchild;
	T->rchild = TT->lchild;
	TT->lchild = T;

	T->height = MAX(getHeight(T->lchild), getHeight(T->rchild)) + 1; //WT.EDTI
	TT->height = MAX(getHeight(TT->lchild), T->height) + 1;

	return TT;
}
/***************************************************
 *
 *函数功能：LR -旋转先右旋转，再左旋转
 *输入参数：旋转前的根节点
 *返回值： 根节点指针
 *
****************************************************/
static AVLtree *left_right_rotation(AVLtree *T)//*doubleRotateWithLeft(AVLtree *T)
{
	T->lchild = right_right_rotation(T->lchild);

	return left_left_rotation(T);
}
/***************************************************
 *
 *函数名称：
 *函数功能：RL -先左旋转，再次右旋转
 *输入参数：旋转前的根节点
 *返回值：根节点
 *
****************************************************/
static AVLtree *right_left_rotation(AVLtree *T)//doubleRotateWithRight(AVLtree *T)
{
	T->rchild = left_left_rotation(T->rchild);

	return right_right_rotation(T);
}
/*************************************************************
 *
 *函数名：
 *函数功能：构建一个新的AVL树
 *输入参数：AVL树 T ,插入的树的数据，data
 *返回值：根结点
 *
**************************************************************/
AVLtree *insert(myType data, AVLtree *T)
{
	if(T == NULL) {
		T = (AVLtree *)malloc(sizeof(struct treeNode));
		T->element = data;
		T->lchild = NULL;
		T->rchild = NULL;
		T->height = 0;
	}
	else if (data < T->element)
	 {
		T->lchild = insert(data, T->lchild);
		if(GET_HEIGHT(T->lchild) - GET_HEIGHT(T->rchild) == 2)
        { //WT.EDIT

			if(data < T->lchild->element)
				T = left_left_rotation(T);
			else
				T = left_right_rotation(T);//doubleRotateWithLeft(T);
        }
	}
	else if (data > T->element)
    {
		T->rchild = insert(data, T->rchild);
		if(GET_HEIGHT(T->rchild) - GET_HEIGHT(T->lchild) == 2)
        {  //WT.EDIT
          if(data > T->rchild->element)
				T = right_right_rotation(T);
			else
				T = right_left_rotation(T);//doubleRotateWithRight(T);
        }

	}

	T->height = MAX(getHeight(T->lchild), getHeight(T->rchild)) + 1; //WT.EDIT

	return T;
}
/****************************************
 *
 *函数功能：删除节点
 *
 *
 *
*****************************************/
AVLtree *erase(myType data, AVLtree *T)
{
	AVLtree *tmpNode;

	if(T == NULL) {
		printf("NOT FOUNT\n");
	} else if (data < T->element) {
		T->lchild = erase(data, T->lchild);
		if(getHeight(T->rchild) - getHeight(T->lchild) == 2) {
			AVLtree *tmp = T->rchild;
			if(getHeight(tmp->lchild) > getHeight(tmp->rchild))
				T = right_left_rotation(T);//doubleRotateWithRight(T);
			else
				T = right_right_rotation(T);
		}
	} else if (data > T->element) {
		T->rchild = erase(data, T->rchild);
		if(getHeight(T->lchild) - getHeight(T->rchild) == 2) {
			AVLtree *tmp = T->lchild;
			if(getHeight(tmp->rchild) > getHeight(tmp->lchild))
				T = left_right_rotation(T);//doubleRotateWithLeft(T);
			else
				T = left_left_rotation(T);
		}
	//found element to delete, two children
	} else if (T->lchild && T->rchild){
		if(getHeight(T->rchild) > getHeight(T->lchild)) {
			tmpNode = findMin(T->rchild);//将右子树的最小值代替root
			T->element = tmpNode->element;
			T->rchild = erase(T->element, T->rchild);
		} else {
			tmpNode = findMax(T->lchild);//将左子树的最大值代替root
			T->element = tmpNode->element;
			T->lchild = erase(T->element, T->lchild);
		}
	} else {
		//one or zero children
		tmpNode = T;
		T = (T->lchild == NULL ? T->rchild : T->lchild);
		free(tmpNode);
	}

	return T;
}
/******************************************
 *
 *层次遍历
 *
 *
*******************************************/
void levelOrder(AVLtree *T)
{
	QUEUE *q = createQueue(100);

	while(T != NULL) {
		printf("%d ", T->element);
		if(T->lchild != NULL)
			enQueue(T->lchild, q);
		if(T->rchild != NULL)
			enQueue(T->rchild, q);

		if(!isEmpty(q))
			T = frontAndDequeue(q);
		else
			T = NULL;
	}

	disposeQueue(q);
}
/**************************************************
 *
 *函数名称:
 *函数功能：先序遍历：根---左树---右树
 *输入参数：树的结构指针
 *
 *
***************************************************/
void preOrder(AVLtree *T)
{
	if(T != NULL) {
		printf("%d ", T->element);
		preOrder(T->lchild);
		preOrder(T->rchild);
	}
}
/**************************************************
 *
 *函数名称:
 *函数功能：中序遍历--左树--根--右树
 *输入参数：树的结构指针
 *返回值：无
 *
***************************************************/
void inOrder(AVLtree *T)
{
	if(T != NULL) {
		inOrder(T->lchild);
		printf("%d ", T->element);
		inOrder(T->rchild);
	}
}
/**************************************************
 *
 *函数名称:
 *函数功能：后序遍历--左树--右树---根
 *输入参数：树的结构指针
 *返回值：无
 *
***************************************************/
void postOrder(AVLtree *T)
{
	if(T != NULL) {
		postOrder(T->lchild);
		postOrder(T->rchild);
		printf("%d ", T->element);
	}
}
/*
void createTree(AVLtree **T)
{
	myType data;
	scanf("%d", &data);

	if(data == -1) {
		*T = NULL;
	} else {
		*T = (AVLtree *)malloc(sizeof(struct treeNode));
		(*T)->element = data;
		printf("请输入%d的左孩子节点：", data);
		createTree(&((*T)->lchild));
		printf("请输入%d的右孩子节点：", data);
		createTree(&((*T)->rchild));
	}
}
*/
/******************************************************
 *
 *函数功能：打印AVL树
 *参数：tree AVL树的节点， key --节点的键值
 *    direction --- 0. 表示该节点的根结点
 *                  -1，表示该节点是它的父结点的左孩子
 *                   1,表示该结点是它的父结点的右孩子
 *
*********************************************************/
void printf_avltree(AVLtree *tree,myType key,int direction)
{
   if(tree !=NULL)
   {
       if(direction == 0) //tree 是根结点
           printf("%2d is root，%d \n",tree ->element,key);
       else // tree 是分支节点
           printf("%2d is %2d's %6s child\n",tree->element,key,direction==1?"right":"left");
           printf_avltree(tree->lchild,tree->element,-1);
           printf_avltree(tree->rchild,tree->element,1);
   }

}

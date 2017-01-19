#include <stdio.h>
#include <stdlib.h>

struct node{
    int a[3][3];
    int g ;
    struct node *nxt[4];
    struct node *anc;
    int count;
    int cost;

};
typedef struct node node;
FILE *fp;
int expanded_node = 0;
int generated_node=0;
struct queue_p{
    struct node *data;
    struct queue_p *nxt;
    };
struct queue_p *top;
int q_count;
struct queue_p *visited_nodes;
void push(node*);
node* pop();

struct stack_p{
    struct node* data;
struct stack_p* nxt;
struct stack_p* bck;
};

struct stack_p *path =NULL;
void initi(node*,node*);
void display(node*);
void succ(node*,node*);
void cpy();
int stcmp(node*,node*);
int manhattan(node*,node*);
int g_cost(node*,node*);

void push_visit(node*);

int main()
{
    int f=1,k=0;
    node* ptr;
    node* goal;
    node* temp;
    struct stack_p* rear = NULL;
    ptr  = (struct node*) malloc(sizeof(struct node));
    goal  = (struct node*) malloc(sizeof(struct node));
    initi(ptr,goal);

    push(ptr);

   //Start the Game tree

     while(f)
     {
        k=0;

        temp = pop();
        if(temp==NULL)
        {
            printf("\nFatal error\n");
            break;
        }

      //  display(temp);
        printf("\n");
        succ(temp,goal);
        expanded_node++;
        if(goal_cond(goal,temp)==1)
            {
                printf("Found");
                f=0;

            }
            else
            {

                while(k<temp->count)
                {

                   if(temp->anc!=NULL)
                    {

                        if(stcmp(temp->nxt[k],temp->anc)!=1)
                        {
                            push(temp->nxt[k]);
                        }

                    }
                    else{

                                push(temp->nxt[k]);

                    }

                    k++;
                }

            }


   }

    fprintf(fp,"Number of nodes generated = %d\nNumber of Nodes Expanded = %d\nBest Solution Path:\n",generated_node,expanded_node);
    path = (struct stack_p*) malloc(sizeof(struct stack_p));
    path->data = temp;
    path->bck = NULL;
    path->nxt = NULL;
    temp = temp->anc;
   while(temp!=NULL)
   {
        path->nxt = (struct stack_p*) malloc(sizeof(struct stack_p));
        rear = path;
        path = path->nxt;
        path->data = temp;
        path->bck = rear;
       temp = temp->anc;
   }

    while(path!=NULL)
    {
        display(path->data);
        path = path->bck;
    }


    return 0;
}

void initi(node* ptr, node* goal)
{
    int i,j;
    FILE *fo;
    path = NULL;
    top = NULL;
    q_count = 0;
    fo = fopen("input.txt","r");
    fp = fopen("output.txt","w");
    visited_nodes = NULL;
    //Input for Initial State. Input is take from Input.txt file.
    for(i=0;i<3;i++)
    {

            fscanf(fo,"%d %d %d",&ptr->a[i][0],&ptr->a[i][1],&ptr->a[i][2]);
    }
    ptr->g = 0;
    //Set initial cost and Depth to 0 as this is root node.
    ptr->count = 0;
    ptr->nxt[0]=NULL;
    ptr->nxt[1]=NULL;
    ptr->nxt[2]=NULL;
    ptr->nxt[3]=NULL;
    ptr->anc = NULL;

    //Input for Goal State. Goal State is taken from Input.txt
    goal->g = 0;
    for(i=0;i<3;i++)
    {

            fscanf(fo,"%d %d %d",&goal->a[i][0],&goal->a[i][1],&goal->a[i][2]);
    }
    //Set Initial Cost and Depth as 0 as this is Goal node.
    goal->count = 0;
    goal->nxt[0]=NULL;
    goal->nxt[1]=NULL;
    goal->nxt[2]=NULL;
    goal->nxt[3]=NULL;
    goal->anc = NULL;
    ptr->cost = ptr->g+manhattan(ptr,goal);

}

//This Function Displays the Board, which is passed as an Argument. The Output is written in Output.txt file.
void display(node* ptr)
{
    int i,j;
    node* temp;

    for(i =0;i<3;i++)
    {
        fprintf(fp,"%d %d %d\n",ptr->a[i][0],ptr->a[i][1],ptr->a[i][2]);

    }
    fprintf(fp,"\n");
    j=0;

}

//This function is Responsible for Generation of Successor node for a given node.
void succ(node* ptr, node* goal)
{
    int i,j;
    int found = 0;
    node* temp;
    for(i = 0;i<3;i++)
    {
        for(j = 0;j<3;j++)
        {
            if(ptr->a[i][j]==0)
            {
                found = 1;
                break;
            }

        }
        if(found == 1)
            break;
    }

    //Successor 1 Generation and set the Dept to 1+that of Parent. Set the Cost = g(n) + h(n)
    temp  = (struct node*) malloc(sizeof(struct node));
    cpy(temp,ptr);
    if(i+1 <= 2)
    {
        temp->a[i][j] = temp->a[i+1][j];
        temp->a[i+1][j] = 0;
        temp->count = 0;
        temp->g = ptr->g+1;
        temp->nxt[0] = NULL;
        temp->nxt[1] = NULL;
        temp->nxt[2] = NULL;
        temp->nxt[3] = NULL;
        temp->cost = g_cost(temp,goal);
        temp->anc = ptr;
        ptr->nxt[ptr->count++] = temp;
        generated_node++;

    }

    //Successor 2 Generation and set the Dept to 1+that of Parent. Set the Cost = g(n) + h(n)
    temp  = (struct node*) malloc(sizeof(struct node));
    cpy(temp,ptr);
    if(i-1>=0)
    {
        temp->a[i][j] = temp->a[i-1][j];
        temp->a[i-1][j] = 0;
        temp->count = 0;
        temp->g = ptr->g+1;
        temp->nxt[0] = NULL;
        temp->nxt[1] = NULL;
        temp->nxt[2] = NULL;
        temp->nxt[3] = NULL;
        temp->anc = ptr;
        temp->cost = g_cost(temp,goal);
        ptr->nxt[ptr->count++] = temp;
        generated_node++;
    }

    //Successor 3 Generation and set the Dept to 1+that of Parent. Set the Cost = g(n) + h(n)
    temp  = (struct node*) malloc(sizeof(struct node));
    cpy(temp,ptr);
    if(j+1<=2)
    {
        temp->a[i][j] = temp->a[i][j+1];
        temp->a[i][j+1] = 0;
        temp->count = 0;
        temp->g = ptr->g+1;
        temp->nxt[0] = NULL;
        temp->nxt[1] = NULL;
        temp->nxt[2] = NULL;
        temp->nxt[3] = NULL;
        temp->anc = ptr;
        temp->cost = g_cost(temp,goal);
        ptr->nxt[ptr->count++] = temp;

        generated_node++;
    }

    //Successor 4 Generation and set the Dept to 1+that of Parent. Set the Cost = g(n) + h(n)
    temp  = (struct node*) malloc(sizeof(struct node));
    cpy(temp,ptr);
    if(j-1>=0)
    {
        temp->a[i][j] = temp->a[i][j-1];
        temp->a[i][j-1] = 0;
        temp->count = 0;
        temp->g = ptr->g+1;
        temp->nxt[0] = NULL;
        temp->nxt[1] = NULL;
        temp->nxt[2] = NULL;
        temp->nxt[3] = NULL;
        temp->anc = ptr;
        temp->cost = g_cost(temp,goal);
        ptr->nxt[ptr->count++] = temp;
        generated_node++;

    }
}

int g_cost(node* temp, node* goal)
{
    return(temp->g + manhattan(temp,goal));
}

//This function is Used for copying the value of 1 node to Other
void cpy(node* temp, node* ptr)
{
    int i,j;
    for(i = 0;i<3;i++)
    {
        for(j = 0;j<3;j++)
        {
            temp->a[i][j] = ptr->a[i][j];

        }

    }
}

//This function is used for Comparison of One Node to another.
int stcmp(node* ptr, node* temp)
{
    int i,j;

    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            if(ptr->a[i][j]!=temp->a[i][j])
            {

                return 0;
            }

        }
    }

    return 1;
}

//This Function is Responsible for Cal calculation of Heuristic function h(n)
int manhattan(node *ptr, node *temp)
{
    int i=0,j=0,found=0,sum=0,k=0,l=0;
    for(i = 0;i<3;i++)
    {
        for(j=0;j<3;j++)
        {
            if(ptr->a[i][j]==0)
            {

            }
            else
            {

            for(k = 0;k<3;k++)
            {

                for(l = 0;l<3;l++)
                {
                    if(ptr->a[i][j]==temp->a[k][l])
                    {

                        found = 1;
                        break;

                    }

                }
                if(found==1)
                    {
                        found=0;
                        break;
                    }
            }
            sum = sum + (abs(k-i) + abs(l-j));

            }


        }

    }
    return sum;
}

//This is the Queue Push function. Based on the Cost the push() function will inert the node such that all nodes with greater cost are below that node.
void push(node* temp)
{
   struct queue_p *ptr , *ptr_bck, *t;

    if(top==NULL)
    {
        top = (struct queue_p*) malloc(sizeof(struct queue_p));
        top->data = temp;
        top->nxt=NULL;
    }
    else
    {
        ptr = top;
        while(ptr!=NULL)
        {
            if(stcmp(ptr,temp)==1)
            {
                if(ptr->data->cost > temp->cost)
                {
                    ptr->data = temp;
                    return;
                }
                else
                    return;

            }
            ptr = ptr->nxt;
        }


            if(temp->cost < top->data->cost)
            {
                ptr = (struct queue_p*) malloc(sizeof(struct queue_p));
                ptr->nxt = top;
                ptr->data = temp;
                top = ptr;

            }
            else
            {
            //Check which node cost is greater than temp node
                ptr = top;
                while(ptr!=NULL)
                {
                    if(temp->cost < ptr->data->cost)
                {
                    break;
                }
                ptr_bck = ptr;
                ptr = ptr->nxt;
                }
            //if ptr==NULL than we are at end of list and hence proceed with insertion at end of list
                if(ptr==NULL)
                {

                ptr_bck->nxt = (struct queue_p*) malloc(sizeof(struct queue_p));
                ptr_bck = ptr_bck->nxt;
                ptr_bck->data = temp;
                ptr_bck->nxt = NULL;

                }
                else{

                    //Inert the node in between two elements
                    ptr_bck->nxt = (struct queue_p*) malloc(sizeof(struct queue_p));
                    ptr_bck = ptr_bck->nxt;
                    ptr_bck->nxt = ptr;
                    ptr_bck->data = temp;
                }

            }




    }
    q_count++;

}

//This Function checks the Goal condition
int goal_cond(node* ptr, node* temp)
{

    if(manhattan(ptr,temp)==0)
        return 1;
    else
        return 0;
}

//This function is used to return the top value of Queue
node* pop()
{
    q_count--;
   // printf("%d  ",q_count);
    node* temp;
    if(top==NULL)
        return NULL;
    temp = top->data;
    top = top->nxt;
    return temp;
}


//This function is used to add visited nodes in Visited nodes list.
void push_visit(node* temp)
{
    struct queue_p *ptr;
     if(visited_nodes==NULL)
    {
        visited_nodes = (struct queue_p*) malloc(sizeof(struct queue_p));
        visited_nodes->data = temp;
        visited_nodes->nxt=NULL;
    }
    else
    {
        ptr = visited_nodes;
        while(ptr->nxt!=NULL)
            ptr = ptr->nxt;

        ptr->nxt = (struct queue_p*) malloc(sizeof(struct queue_p));
        ptr = ptr->nxt;
        ptr->nxt = NULL;
        ptr->data = temp;
    }
}

// This function checks whether a give node matches the nodes in Visited nodes queue.
int check_visited_nodes(node* temp)
{

    struct queue_p* ptr;
    ptr = visited_nodes;
    if(ptr==NULL)
        return 0;

    while(ptr->nxt!=NULL)
    {
        if(stcmp(temp,ptr->data)==1)
            return 1;

        ptr = ptr->nxt;
    }

    return 0;
}

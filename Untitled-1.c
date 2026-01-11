#include <stdio.h>
#include <string.h>

typedef struct Student {
    char fname[20];
    char lname[20];
    char id[10];
    float golch;
} Student;


void read_students  (Student a[], int n);
void print_students (Student a[], int n);
void print          (Student st);
int  search_by_fname(Student a[], int n, char fname[]);
int  search_by_lname(Student a[], int n, char lname[]);
int  search_by_id   (Student a[], int n, char id[]);
int  search_by_golch(Student a[], int n, float golch);
void sort_by_golch  (Student a[], int n);

int main(void)
{
    Student a[100];
    int n, cmd, idx;

    printf("Oyutnii too: ");
    scanf("%d", &n);

    read_students(a, n);
    print_students(a, n);

    char fname[20], lname[20], id[20];
    float golch;

    while (1) {
        printf(
            "1: Nereer xaix, 2: Ovgoor xaix, 3: ID-aar xaix, 4: Golchoor xaix,\n"
            "5: Golchoor erembelex, 6: Xevlex, 7: Garax\n> "
        );
        scanf("%d", &cmd);

        if (cmd == 1) {
            printf("Xaix ner: ");
            scanf("%20s", fname);
            idx = search_by_fname(a, n, fname);
            if (idx == -1)  printf("Oyutan oldsongui\n");
            else            print(a[idx]);

        } else if (cmd == 2) {
            printf("Xaix ovog: ");
            scanf("%20s", lname);
            idx = search_by_lname(a, n, lname);
            if (idx == -1)  printf("Oyutan oldsongui\n");
            else            print(a[idx]);

        } else if (cmd == 3) {
            printf("Xaix id: ");
            scanf("%10s", id);
            idx = search_by_id(a, n, id);
            if (idx == -1)  printf("Oyutan oldsongui\n");
            else            print(a[idx]);

        } else if (cmd == 4) {
            printf("Xaix golch: ");
            scanf("%f", &golch);
            idx = search_by_golch(a, n, golch);
            if (idx == -1)  printf("Oyutan oldsongui\n");
            else            print(a[idx]);

        } else if (cmd == 5) {
            sort_by_golch(a, n);
            if 

        } else if (cmd == 6) {
            print_students(a, n);

        } else {
            break;
        }
    }

    return 0;
}

// n oytanii medeelel avna
void read_students(Student a[], int n) {
    for (int i = 0; i < n; i++) {
        printf("Student %d — firstname, lastname, id, golch: ", i+1);
        scanf("%20s%20s%9s%f",
              a[i].fname,
              a[i].lname,
              a[i].id,
              &a[i].golch);
    }
}

// oytan bug hevleh 
void print_students(Student a[], int n) {
    for (int i = 0; i < n; i++) {
        printf("\n--- Oytan %d ---\n", i);
        print(a[i]);
    }
}

// oytan hev
void print(Student st) {
    printf("Ner   : %s\n", st.fname);
    printf("Ovog  : %s\n", st.lname);
    printf("ID    : %s\n", st.id);
    printf("Golch : %.2f\n", st.golch);
}

// ner haih
int search_by_fname(Student a[], int n, char fname[]) {
    for (int i = 0; i < n; i++)
        if (strcmp(a[i].fname, fname) == 0)
            return i;
    return -1;
}

// Ovog haih
int search_by_lname(Student a[], int n, char lname[]) {
    for (int i = 0; i < n; i++)
        if (strcmp(a[i].lname, lname) == 0)
            return i;
    return -1;
}

// ID haih
int search_by_id(Student a[], int n, char id[]) {
    for (int i = 0; i < n; i++)
        if (strcmp(a[i].id, id) == 0)
            return i;
    return -1;
}

// golch haih
int search_by_golch(Student a[], int n, float golch) {
    for (int i = 0; i < n; i++)
        if (a[i].golch == golch)
            return i;
    return -1;
}

// golch erembe
void sort_by_golch(Student a[], int n) {
    for (int i = 0; i < n - 1; i++) {
        int min_idx = i;
        for (int j = i + 1; j < n; j++) {
            if (a[j].golch < a[min_idx].golch)
                min_idx = j;
        }
        if (min_idx != i) {
            Student tmp = a[i];
            a[i] = a[min_idx];
            a[min_idx] = tmp;
        }
    }
}
